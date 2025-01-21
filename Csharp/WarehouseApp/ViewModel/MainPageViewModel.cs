using WarehouseApp.Model;
using System.Collections.ObjectModel;

namespace WarehouseApp.ViewModel;

public class MainPageViewModel
{
    public ObservableCollection<CardViewModel> Items { get; }

    public MainPageViewModel()
    {
        Items = new ObservableCollection<CardViewModel>
        {
            new CardViewModel("Прокладки", "prokladki.png", new Component("Прокладки")),
            new CardViewModel("Болты", "bolt.png", new Component("Болты")),
            new CardViewModel("Гайки", "gayki.png", new Component("Гайки")),
            new CardViewModel("Бруски", "brusok.png", new Component("Бруски")),
            new CardViewModel("Шайбы", "shayba.png", new Component("Шайбы")),
        };
    }

    public IEnumerable<Component> GetComponents()
    {
        return Items.Select(item => item.Component);
    }
}