using System.Collections.ObjectModel;
using System.Windows.Input;

namespace WarehouseApp.ViewModel
{
    public class OrderPageViewModel
    {
        public ObservableCollection<OrderCardViewModel> Items { get; }
        public ICommand CreateOrder { get; }

        public OrderPageViewModel()
        {
            Items = new ObservableCollection<OrderCardViewModel>();
            CreateOrder = new Command(async () => await CreateOrderAsync());
        }

        // Асинхронный метод для загрузки элементов
        public async Task LoadItemsAsync()
        {
            var appState = await AppState.GetInstanceAsync(); // Получаем инстанс AppState асинхронно

            var updatedItems = appState.Goods?
                .Where(goods => goods.Quantity > 0)
                .Select(goods => new OrderCardViewModel(goods)) ?? Enumerable.Empty<OrderCardViewModel>();

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

            var updatedItems = appState.Goods?
                .Where(goods => goods.Quantity > 0)
                .Select(goods => new OrderCardViewModel(goods)) ?? Enumerable.Empty<OrderCardViewModel>();

            Items.Clear();
            foreach (var item in updatedItems)
            {
                Items.Add(item);
            }
        }

        // Асинхронный метод для создания заказа
        private async Task CreateOrderAsync()
        {
            var appState = await AppState.GetInstanceAsync(); // Получаем инстанс AppState асинхронно
            var quantities = GetItemsCount(appState);
            await appState.EditOrderAsync(quantities); // Создаем заказ асинхронно
        }

        // Получение количества товаров
        private static int[] GetItemsCount(AppState appState)
        {
            return appState.Goods?
                .Select(goods => goods.Quantity)
                .Take(5)
                .ToArray() ?? [];
        }
    }
}
