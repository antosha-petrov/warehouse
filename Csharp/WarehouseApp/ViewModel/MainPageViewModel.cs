using System.Collections.ObjectModel;
using System.Windows.Input;

namespace WarehouseApp.ViewModel
{
    public class MainPageViewModel
    {
        public ObservableCollection<CardViewModel> Items { get; }
        public ICommand NavigateToOrder { get; }

        public MainPageViewModel()
        {
            Items = new ObservableCollection<CardViewModel>();
            NavigateToOrder = new Command(() => Shell.Current.GoToAsync("//Order"));
        }

        // Асинхронный метод для загрузки элементов
        public async Task LoadItemsAsync()
        {
            var appState = await AppState.GetInstanceAsync(); // Получаем инстанс AppState асинхронно

            var updatedItems = appState.Goods?.Select(goods => new CardViewModel(goods)) ?? Enumerable.Empty<CardViewModel>();

            Items.Clear();
            foreach (var item in updatedItems)
            {
                Items.Add(item);
            }
        }

        // Асинхронный метод для обновления элементов
        public async Task RefreshItemsAsync()
        {
            var appState = await AppState.GetInstanceAsync(); // Получаем инстанс AppState асинхронно

            var updatedItems = appState.Goods?.Select(goods => new CardViewModel(goods)) ?? Enumerable.Empty<CardViewModel>();

            Items.Clear();
            foreach (var item in updatedItems)
            {
                Items.Add(item);
            }
        }
    }
}
