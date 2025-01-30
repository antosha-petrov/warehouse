using System.Collections.ObjectModel;
using System.Windows.Input;
using WarehouseApp.Services;

namespace WarehouseApp.ViewModel;

public class MainPageViewModel
{
    public ObservableCollection<CardViewModel> Items { get; }

    public MainPageViewModel()
    {
        Items = new ObservableCollection<CardViewModel>(
            AppState.Instance.Goods
            .Select(goods => new CardViewModel(goods))
        );

        NavigateToOrder = new Command(() => Shell.Current.GoToAsync("//Order"));
    }

    public void RefreshItems()
    {
        Items.Clear();

        var updatedItems = AppState.Instance.Goods
            .Select(goods => new CardViewModel(goods));

        foreach (var item in updatedItems)
        {
            Items.Add(item);
        }
    }

    public ICommand NavigateToOrder { get; }
}